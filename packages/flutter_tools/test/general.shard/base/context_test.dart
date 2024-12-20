// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter_tools/src/base/context.dart';

import '../../src/common.dart';

void main() {
  group('AppContext', () {
    group('global getter', () {
      late bool called;

      setUp(() {
        called = false;
      });

      test('returns non-null context in the root zone', () {
        expect(context, isNotNull);
      });

      test('returns root context in child of root zone if zone was manually created', () {
        final Zone rootZone = Zone.current;
        final AppContext rootContext = context;
        runZoned<void>(() {
          expect(Zone.current, isNot(rootZone));
          expect(Zone.current.parent, rootZone);
          expect(context, rootContext);
          called = true;
        });
        expect(called, isTrue);
      });

      test('returns child context after run', () async {
        final AppContext rootContext = context;
        await rootContext.run<void>(
          name: 'child',
          body: () {
            expect(context, isNot(rootContext));
            expect(context.name, 'child');
            called = true;
          },
        );
        expect(called, isTrue);
      });

      test('returns grandchild context after nested run', () async {
        final AppContext rootContext = context;
        await rootContext.run<void>(
          name: 'child',
          body: () async {
            final AppContext childContext = context;
            await childContext.run<void>(
              name: 'grandchild',
              body: () {
                expect(context, isNot(rootContext));
                expect(context, isNot(childContext));
                expect(context.name, 'grandchild');
                called = true;
              },
            );
          },
        );
        expect(called, isTrue);
      });

      test('scans up zone hierarchy for first context', () async {
        final AppContext rootContext = context;
        await rootContext.run<void>(
          name: 'child',
          body: () {
            final AppContext childContext = context;
            runZoned<void>(() {
              expect(context, isNot(rootContext));
              expect(context, same(childContext));
              expect(context.name, 'child');
              called = true;
            });
          },
        );
        expect(called, isTrue);
      });
    });

    group('operator[]', () {
      test('still finds values if async code runs after body has finished', () async {
        final Completer<void> outer = Completer<void>();
        final Completer<void> inner = Completer<void>();
        String? value;
        await context.run<void>(
          body: () {
            outer.future.then<void>((_) {
              value = context.get<String>();
              inner.complete();
            });
          },
          fallbacks: <Type, Generator>{String: () => 'value'},
        );
        expect(value, isNull);
        outer.complete();
        await inner.future;
        expect(value, 'value');
      });

      test('caches generated override values', () async {
        int consultationCount = 0;
        String? value;
        await context.run<void>(
          body: () async {
            final StringBuffer buf = StringBuffer(context.get<String>()!);
            buf.write(context.get<String>());
            await context.run<void>(
              body: () {
                buf.write(context.get<String>());
              },
            );
            value = buf.toString();
          },
          overrides: <Type, Generator>{
            String: () {
              consultationCount++;
              return 'v';
            },
          },
        );
        expect(value, 'vvv');
        expect(consultationCount, 1);
      });

      test('caches generated fallback values', () async {
        int consultationCount = 0;
        String? value;
        await context.run(
          body: () async {
            final StringBuffer buf = StringBuffer(context.get<String>()!);
            buf.write(context.get<String>());
            await context.run<void>(
              body: () {
                buf.write(context.get<String>());
              },
            );
            value = buf.toString();
          },
          fallbacks: <Type, Generator>{
            String: () {
              consultationCount++;
              return 'v';
            },
          },
        );
        expect(value, 'vvv');
        expect(consultationCount, 1);
      });

      test('returns null if generated value is null', () async {
        final String? value = await context.run<String?>(
          body: () => context.get<String>(),
          overrides: <Type, Generator>{String: () => null},
        );
        expect(value, isNull);
      });

      test('throws if generator has dependency cycle', () async {
        final Future<String?> value = context.run<String?>(
          body: () async {
            return context.get<String>();
          },
          fallbacks: <Type, Generator>{
            int: () => int.parse(context.get<String>() ?? ''),
            String: () => '${context.get<double>()}',
            double: () => context.get<int>()! * 1.0,
          },
        );
        expect(
          () => value,
          throwsA(
            isA<ContextDependencyCycleException>()
                .having((ContextDependencyCycleException error) => error.cycle, 'cycle', <Type>[
                  String,
                  double,
                  int,
                ])
                .having(
                  (ContextDependencyCycleException error) => error.toString(),
                  'toString()',
                  'Dependency cycle detected: String -> double -> int',
                ),
          ),
        );
      });
    });

    group('run', () {
      test('returns the value returned by body', () async {
        expect(await context.run<int>(body: () => 123), 123);
        expect(await context.run<String>(body: () => 'value'), 'value');
        expect(await context.run<int>(body: () async => 456), 456);
      });

      test('passes name to child context', () async {
        await context.run<void>(
          name: 'child',
          body: () {
            expect(context.name, 'child');
          },
        );
      });

      group('fallbacks', () {
        late bool called;

        setUp(() {
          called = false;
        });

        test('are applied after parent context is consulted', () async {
          final String? value = await context.run<String?>(
            body: () {
              return context.run<String?>(
                body: () {
                  called = true;
                  return context.get<String>();
                },
                fallbacks: <Type, Generator>{String: () => 'child'},
              );
            },
          );
          expect(called, isTrue);
          expect(value, 'child');
        });

        test('are not applied if parent context supplies value', () async {
          bool childConsulted = false;
          final String? value = await context.run<String?>(
            body: () {
              return context.run<String?>(
                body: () {
                  called = true;
                  return context.get<String>();
                },
                fallbacks: <Type, Generator>{
                  String: () {
                    childConsulted = true;
                    return 'child';
                  },
                },
              );
            },
            fallbacks: <Type, Generator>{String: () => 'parent'},
          );
          expect(called, isTrue);
          expect(value, 'parent');
          expect(childConsulted, isFalse);
        });

        test('may depend on one another', () async {
          final String? value = await context.run<String?>(
            body: () {
              return context.get<String>();
            },
            fallbacks: <Type, Generator>{int: () => 123, String: () => '-${context.get<int>()}-'},
          );
          expect(value, '-123-');
        });
      });

      group('overrides', () {
        test('intercept consultation of parent context', () async {
          bool parentConsulted = false;
          final String? value = await context.run<String?>(
            body: () {
              return context.run<String?>(
                body: () => context.get<String>(),
                overrides: <Type, Generator>{String: () => 'child'},
              );
            },
            fallbacks: <Type, Generator>{
              String: () {
                parentConsulted = true;
                return 'parent';
              },
            },
          );
          expect(value, 'child');
          expect(parentConsulted, isFalse);
        });
      });
    });
  });
}
