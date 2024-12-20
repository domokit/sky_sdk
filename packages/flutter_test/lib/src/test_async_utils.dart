// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'widget_tester.dart';
library;

import 'dart:async';

import 'package:flutter/foundation.dart';

class _AsyncScope {
  _AsyncScope(this.creationStack, this.zone);
  final StackTrace creationStack;
  final Zone zone;
}

// Examples can assume:
// late WidgetTester tester;

/// Utility class for all the async APIs in the `flutter_test` library.
///
/// This class provides checking for asynchronous APIs, allowing the library to
/// verify that all the asynchronous APIs are properly `await`ed before calling
/// another.
///
/// For example, it prevents this kind of code:
///
/// ```dart
/// tester.pump(); // forgot to call "await"!
/// tester.pump();
/// ```
///
/// ...by detecting, in the second call to `pump`, that it should actually be:
///
/// ```dart
/// await tester.pump();
/// await tester.pump();
/// ```
///
/// It does this while still allowing nested calls, e.g. so that you can
/// call [expect] from inside callbacks.
///
/// You can use this in your own test functions, if you have some asynchronous
/// functions that must be used with "await". Wrap the contents of the function
/// in a call to TestAsyncUtils.guard(), as follows:
///
/// ```dart
/// Future<void> myTestFunction() => TestAsyncUtils.guard(() async {
///   // ...
/// });
/// ```
abstract final class TestAsyncUtils {
  static const String _className = 'TestAsyncUtils';

  static final List<_AsyncScope> _scopeStack = <_AsyncScope>[];

  /// Calls the given callback in a new async scope. The callback argument is
  /// the asynchronous body of the calling method. The calling method is said to
  /// be "guarded". Nested calls to guarded methods from within the body of this
  /// one are fine, but calls to other guarded methods from outside the body of
  /// this one before this one has finished will throw an exception.
  ///
  /// This method first calls [guardSync].
  static Future<T> guard<T>(Future<T> Function() body) {
    guardSync();
    final Zone zone = Zone.current.fork(
      zoneValues: <dynamic, dynamic>{
        _scopeStack: true, // so we can recognize this as our own zone
      },
    );
    final _AsyncScope scope = _AsyncScope(StackTrace.current, zone);
    _scopeStack.add(scope);
    final Future<T> result = scope.zone.run<Future<T>>(body);
    late T resultValue; // This is set when the body of work completes with a result value.
    Future<T> completionHandler(dynamic error, StackTrace? stack) {
      assert(_scopeStack.isNotEmpty);
      assert(_scopeStack.contains(scope));
      bool leaked = false;
      _AsyncScope closedScope;
      final List<DiagnosticsNode> information = <DiagnosticsNode>[];
      while (_scopeStack.isNotEmpty) {
        closedScope = _scopeStack.removeLast();
        if (closedScope == scope) {
          break;
        }
        if (!leaked) {
          information.add(ErrorSummary('Asynchronous call to guarded function leaked.'));
          information.add(ErrorHint('You must use "await" with all Future-returning test APIs.'));
          leaked = true;
        }
        final _StackEntry? originalGuarder = _findResponsibleMethod(
          closedScope.creationStack,
          'guard',
          information,
        );
        if (originalGuarder != null) {
          information.add(
            ErrorDescription(
              'The test API method "${originalGuarder.methodName}" '
              'from class ${originalGuarder.className} '
              'was called from ${originalGuarder.callerFile} '
              'on line ${originalGuarder.callerLine}, '
              'but never completed before its parent scope closed.',
            ),
          );
        }
      }
      if (leaked) {
        if (error != null) {
          information.add(
            DiagnosticsProperty<dynamic>(
              'An uncaught exception may have caused the guarded function leak. The exception was',
              error,
              style: DiagnosticsTreeStyle.errorProperty,
            ),
          );
          information.add(
            DiagnosticsStackTrace('The stack trace associated with this exception was', stack),
          );
        }
        throw FlutterError.fromParts(information);
      }
      if (error != null) {
        return Future<T>.error(error! as Object, stack);
      }
      return Future<T>.value(resultValue);
    }

    return result.then<T>((T value) {
      resultValue = value;
      return completionHandler(null, null);
    }, onError: completionHandler);
  }

  static Zone? get _currentScopeZone {
    Zone? zone = Zone.current;
    while (zone != null) {
      if (zone[_scopeStack] == true) {
        return zone;
      }
      zone = zone.parent;
    }
    return null;
  }

  /// Verifies that there are no guarded methods currently pending (see [guard]).
  ///
  /// If a guarded method is currently pending, and this is not a call nested
  /// from inside that method's body (directly or indirectly), then this method
  /// will throw a detailed exception.
  static void guardSync() {
    if (_scopeStack.isEmpty) {
      // No scopes open, so we must be fine.
      return;
    }
    // Find the current TestAsyncUtils scope zone so we can see if it's the one we expect.
    final Zone? zone = _currentScopeZone;
    if (zone == _scopeStack.last.zone) {
      // We're still in the current scope zone. All good.
      return;
    }
    // If we get here, we know we've got a conflict on our hands.
    // We got an async barrier, but the current zone isn't the last scope that
    // we pushed on the stack.
    // Find which scope the conflict happened in, so that we know
    // which stack trace to report the conflict as starting from.
    //
    // For example, if we called an async method A, which ran its body in a
    // guarded block, and in its body it ran an async method B, which ran its
    // body in a guarded block, but we didn't await B, then in A's block we ran
    // an async method C, which ran its body in a guarded block, then we should
    // complain about the call to B then the call to C. BUT. If we called an async
    // method A, which ran its body in a guarded block, and in its body it ran
    // an async method B, which ran its body in a guarded block, but we didn't
    // await A, and then at the top level we called a method D, then we should
    // complain about the call to A then the call to D.
    //
    // In both examples, the scope stack would have two scopes. In the first
    // example, the current zone would be the zone of the _scopeStack[0] scope,
    // and we would want to show _scopeStack[1]'s creationStack. In the second
    // example, the current zone would not be in the _scopeStack, and we would
    // want to show _scopeStack[0]'s creationStack.
    int skipCount = 0;
    _AsyncScope candidateScope = _scopeStack.last;
    _AsyncScope scope;
    do {
      skipCount += 1;
      scope = candidateScope;
      if (skipCount >= _scopeStack.length) {
        if (zone == null) {
          break;
        }
        // Some people have reported reaching this point, but it's not clear
        // why. For now, just silently return.
        // TODO(ianh): If we ever get a test case that shows how we reach
        // this point, reduce it and report the error if there is one.
        return;
      }
      candidateScope = _scopeStack[_scopeStack.length - skipCount - 1];
    } while (candidateScope.zone != zone);
    final List<DiagnosticsNode> information = <DiagnosticsNode>[
      ErrorSummary('Guarded function conflict.'),
      ErrorHint('You must use "await" with all Future-returning test APIs.'),
    ];
    final _StackEntry? originalGuarder = _findResponsibleMethod(
      scope.creationStack,
      'guard',
      information,
    );
    final _StackEntry? collidingGuarder = _findResponsibleMethod(
      StackTrace.current,
      'guardSync',
      information,
    );
    if (originalGuarder != null && collidingGuarder != null) {
      final String originalKind = originalGuarder.className == null ? 'function' : 'method';
      String originalName;
      if (originalGuarder.className == null) {
        originalName = '$originalKind (${originalGuarder.methodName})';
        information.add(
          ErrorDescription(
            'The guarded "${originalGuarder.methodName}" function '
            'was called from ${originalGuarder.callerFile} '
            'on line ${originalGuarder.callerLine}.',
          ),
        );
      } else {
        originalName = '$originalKind (${originalGuarder.className}.${originalGuarder.methodName})';
        information.add(
          ErrorDescription(
            'The guarded method "${originalGuarder.methodName}" '
            'from class ${originalGuarder.className} '
            'was called from ${originalGuarder.callerFile} '
            'on line ${originalGuarder.callerLine}.',
          ),
        );
      }
      final String again =
          (originalGuarder.callerFile == collidingGuarder.callerFile) &&
                  (originalGuarder.callerLine == collidingGuarder.callerLine)
              ? 'again '
              : '';
      final String collidingKind = collidingGuarder.className == null ? 'function' : 'method';
      String collidingName;
      if ((originalGuarder.className == collidingGuarder.className) &&
          (originalGuarder.methodName == collidingGuarder.methodName)) {
        originalName = originalKind;
        collidingName = collidingKind;
        information.add(
          ErrorDescription(
            'Then, it '
            'was called ${again}from ${collidingGuarder.callerFile} '
            'on line ${collidingGuarder.callerLine}.',
          ),
        );
      } else if (collidingGuarder.className == null) {
        collidingName = '$collidingKind (${collidingGuarder.methodName})';
        information.add(
          ErrorDescription(
            'Then, the "${collidingGuarder.methodName}" function '
            'was called ${again}from ${collidingGuarder.callerFile} '
            'on line ${collidingGuarder.callerLine}.',
          ),
        );
      } else {
        collidingName =
            '$collidingKind (${collidingGuarder.className}.${collidingGuarder.methodName})';
        information.add(
          ErrorDescription(
            'Then, the "${collidingGuarder.methodName}" method '
            '${originalGuarder.className == collidingGuarder.className ? "(also from class ${collidingGuarder.className})" : "from class ${collidingGuarder.className}"} '
            'was called ${again}from ${collidingGuarder.callerFile} '
            'on line ${collidingGuarder.callerLine}.',
          ),
        );
      }
      information.add(
        ErrorDescription(
          'The first $originalName '
          'had not yet finished executing at the time that '
          'the second $collidingName '
          'was called. Since both are guarded, and the second was not a nested call inside the first, the '
          'first must complete its execution before the second can be called. Typically, this is achieved by '
          'putting an "await" statement in front of the call to the first.',
        ),
      );
      if (collidingGuarder.className == null && collidingGuarder.methodName == 'expect') {
        information.add(
          ErrorHint(
            'If you are confident that all test APIs are being called using "await", and '
            'this expect() call is not being called at the top level but is itself being '
            'called from some sort of callback registered before the ${originalGuarder.methodName} '
            'method was called, then consider using expectSync() instead.',
          ),
        );
      }
      information.add(
        DiagnosticsStackTrace(
          '\nWhen the first $originalName was called, this was the stack',
          scope.creationStack,
        ),
      );
    } else {
      information.add(
        DiagnosticsStackTrace(
          '\nWhen the first function was called, this was the stack',
          scope.creationStack,
        ),
      );
    }
    throw FlutterError.fromParts(information);
  }

  /// Verifies that there are no guarded methods currently pending (see [guard]).
  ///
  /// This is used at the end of tests to ensure that nothing leaks out of the test.
  static void verifyAllScopesClosed() {
    if (_scopeStack.isNotEmpty) {
      final List<DiagnosticsNode> information = <DiagnosticsNode>[
        ErrorSummary('Asynchronous call to guarded function leaked.'),
        ErrorHint('You must use "await" with all Future-returning test APIs.'),
      ];
      for (final _AsyncScope scope in _scopeStack) {
        final _StackEntry? guarder = _findResponsibleMethod(
          scope.creationStack,
          'guard',
          information,
        );
        if (guarder != null) {
          information.add(
            ErrorDescription(
              'The guarded method "${guarder.methodName}" '
              '${guarder.className != null ? "from class ${guarder.className} " : ""}'
              'was called from ${guarder.callerFile} '
              'on line ${guarder.callerLine}, '
              'but never completed before its parent scope closed.',
            ),
          );
        }
      }
      throw FlutterError.fromParts(information);
    }
  }

  static bool _stripAsynchronousSuspensions(String line) {
    return line != '<asynchronous suspension>';
  }

  static _StackEntry? _findResponsibleMethod(
    StackTrace rawStack,
    String method,
    List<DiagnosticsNode> information,
  ) {
    assert(method == 'guard' || method == 'guardSync');
    // Web/JavaScript stack traces use a different format.
    if (kIsWeb) {
      return null;
    }
    final List<String> stack =
        rawStack.toString().split('\n').where(_stripAsynchronousSuspensions).toList();
    assert(stack.last == '');
    stack.removeLast();
    final RegExp getClassPattern = RegExp(r'^#[0-9]+ +([^. ]+)');
    Match? lineMatch;
    int index = -1;
    do {
      // skip past frames that are from this class
      index += 1;
      assert(index < stack.length);
      lineMatch = getClassPattern.matchAsPrefix(stack[index]);
      assert(lineMatch != null);
      lineMatch = lineMatch!;
      assert(lineMatch.groupCount == 1);
    } while (lineMatch.group(1) == _className);
    // try to parse the stack to find the interesting frame
    if (index < stack.length) {
      final RegExp guardPattern = RegExp(r'^#[0-9]+ +(?:([^. ]+)\.)?([^. ]+)');
      final Match? guardMatch = guardPattern.matchAsPrefix(
        stack[index],
      ); // find the class that called us
      if (guardMatch != null) {
        assert(guardMatch.groupCount == 2);
        final String? guardClass = guardMatch.group(1); // might be null
        final String? guardMethod = guardMatch.group(2);
        while (index < stack.length) {
          // find the last stack frame that called the class that called us
          lineMatch = getClassPattern.matchAsPrefix(stack[index]);
          if (lineMatch != null) {
            assert(lineMatch.groupCount == 1);
            if (lineMatch.group(1) == (guardClass ?? guardMethod)) {
              index += 1;
              continue;
            }
          }
          break;
        }
        if (index < stack.length) {
          final RegExp callerPattern = RegExp(r'^#[0-9]+ .* \((.+?):([0-9]+)(?::[0-9]+)?\)$');
          final Match? callerMatch = callerPattern.matchAsPrefix(
            stack[index],
          ); // extract the caller's info
          if (callerMatch != null) {
            assert(callerMatch.groupCount == 2);
            final String? callerFile = callerMatch.group(1);
            final String? callerLine = callerMatch.group(2);
            return _StackEntry(guardClass, guardMethod, callerFile, callerLine);
          } else {
            // One reason you might get here is if the guarding method was called directly from
            // a 'dart:' API, like from the Future/microtask mechanism, because dart: URLs in the
            // stack trace don't have a column number and so don't match the regexp above.
            information.add(
              ErrorSummary(
                '(Unable to parse the stack frame of the method that called the method that called $_className.$method(). The stack may be incomplete or bogus.)',
              ),
            );
            information.add(ErrorDescription(stack[index]));
          }
        } else {
          information.add(
            ErrorSummary(
              '(Unable to find the stack frame of the method that called the method that called $_className.$method(). The stack may be incomplete or bogus.)',
            ),
          );
        }
      } else {
        information.add(
          ErrorSummary(
            '(Unable to parse the stack frame of the method that called $_className.$method(). The stack may be incomplete or bogus.)',
          ),
        );
        information.add(ErrorDescription(stack[index]));
      }
    } else {
      information.add(
        ErrorSummary(
          '(Unable to find the method that called $_className.$method(). The stack may be incomplete or bogus.)',
        ),
      );
    }
    return null;
  }
}

class _StackEntry {
  const _StackEntry(this.className, this.methodName, this.callerFile, this.callerLine);
  final String? className;
  final String? methodName;
  final String? callerFile;
  final String? callerLine;
}
