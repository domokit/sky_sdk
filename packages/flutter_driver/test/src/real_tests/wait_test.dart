// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_driver/src/common/wait.dart';

import '../../common.dart';

void main() {
  group('WaitForCondition', () {
    test('WaitForCondition serialize', () {
      expect(const WaitForCondition(NoTransientCallbacks()).serialize(), <String, String>{
        'command': 'waitForCondition',
        'conditionName': 'NoTransientCallbacksCondition',
      });
    });

    test('WaitForCondition serialize with timeout', () {
      expect(
        const WaitForCondition(
          NoTransientCallbacks(),
          timeout: Duration(milliseconds: 10),
        ).serialize(),
        <String, String>{
          'command': 'waitForCondition',
          'timeout': '10',
          'conditionName': 'NoTransientCallbacksCondition',
        },
      );
    });

    test('WaitForCondition deserialize', () {
      final Map<String, String> jsonMap = <String, String>{
        'command': 'waitForCondition',
        'conditionName': 'NoTransientCallbacksCondition',
      };
      final WaitForCondition waitForCondition = WaitForCondition.deserialize(jsonMap);
      expect(waitForCondition.kind, 'waitForCondition');
      expect(waitForCondition.condition, equals(const NoTransientCallbacks()));
    });

    test('WaitForCondition deserialize with timeout', () {
      final Map<String, String> jsonMap = <String, String>{
        'command': 'waitForCondition',
        'timeout': '10',
        'conditionName': 'NoTransientCallbacksCondition',
      };
      final WaitForCondition waitForCondition = WaitForCondition.deserialize(jsonMap);
      expect(waitForCondition.kind, 'waitForCondition');
      expect(waitForCondition.condition, equals(const NoTransientCallbacks()));
      expect(waitForCondition.timeout, equals(const Duration(milliseconds: 10)));
    });

    test('WaitForCondition requiresRootWidget', () {
      expect(const WaitForCondition(NoTransientCallbacks()).requiresRootWidgetAttached, isTrue);
      expect(const WaitForCondition(FirstFrameRasterized()).requiresRootWidgetAttached, isFalse);
    });
  });

  group('NoTransientCallbacksCondition', () {
    test('NoTransientCallbacksCondition serialize', () {
      expect(const NoTransientCallbacks().serialize(), <String, String>{
        'conditionName': 'NoTransientCallbacksCondition',
      });
    });

    test('NoTransientCallbacksCondition deserialize', () {
      final Map<String, String> jsonMap = <String, String>{
        'conditionName': 'NoTransientCallbacksCondition',
      };
      final NoTransientCallbacks condition = NoTransientCallbacks.deserialize(jsonMap);
      expect(condition, equals(const NoTransientCallbacks()));
      expect(condition.serialize(), equals(jsonMap));
    });

    test('NoTransientCallbacksCondition deserialize error', () {
      expect(
        () => NoTransientCallbacks.deserialize(<String, String>{'conditionName': 'Unknown'}),
        throwsA(
          predicate<SerializationException>(
            (SerializationException e) =>
                e.message ==
                'Error occurred during deserializing the NoTransientCallbacksCondition JSON string: {conditionName: Unknown}',
          ),
        ),
      );
    });
  });

  group('NoPendingFrameCondition', () {
    test('NoPendingFrameCondition serialize', () {
      expect(const NoPendingFrame().serialize(), <String, String>{
        'conditionName': 'NoPendingFrameCondition',
      });
    });

    test('NoPendingFrameCondition deserialize', () {
      final Map<String, String> jsonMap = <String, String>{
        'conditionName': 'NoPendingFrameCondition',
      };
      final NoPendingFrame condition = NoPendingFrame.deserialize(jsonMap);
      expect(condition, equals(const NoPendingFrame()));
      expect(condition.serialize(), equals(jsonMap));
    });

    test('NoPendingFrameCondition deserialize error', () {
      expect(
        () => NoPendingFrame.deserialize(<String, String>{'conditionName': 'Unknown'}),
        throwsA(
          predicate<SerializationException>(
            (SerializationException e) =>
                e.message ==
                'Error occurred during deserializing the NoPendingFrameCondition JSON string: {conditionName: Unknown}',
          ),
        ),
      );
    });
  });

  group('FirstFrameRasterizedCondition', () {
    test('FirstFrameRasterizedCondition serialize', () {
      expect(const FirstFrameRasterized().serialize(), <String, String>{
        'conditionName': 'FirstFrameRasterizedCondition',
      });
    });

    test('FirstFrameRasterizedCondition deserialize', () {
      final Map<String, String> jsonMap = <String, String>{
        'conditionName': 'FirstFrameRasterizedCondition',
      };
      final FirstFrameRasterized condition = FirstFrameRasterized.deserialize(jsonMap);
      expect(condition, equals(const FirstFrameRasterized()));
      expect(condition.serialize(), equals(jsonMap));
    });

    test('FirstFrameRasterizedCondition deserialize error', () {
      expect(
        () => FirstFrameRasterized.deserialize(<String, String>{'conditionName': 'Unknown'}),
        throwsA(
          predicate<SerializationException>(
            (SerializationException e) =>
                e.message ==
                'Error occurred during deserializing the FirstFrameRasterizedCondition JSON string: {conditionName: Unknown}',
          ),
        ),
      );
    });

    test('FirstFrameRasterizedCondition requiresRootWidget', () {
      expect(const FirstFrameRasterized().requiresRootWidgetAttached, isFalse);
    });
  });

  group('CombinedCondition', () {
    test('CombinedCondition serialize', () {
      const CombinedCondition combinedCondition = CombinedCondition(<SerializableWaitCondition>[
        NoTransientCallbacks(),
        NoPendingFrame(),
      ]);

      expect(combinedCondition.serialize(), <String, String>{
        'conditionName': 'CombinedCondition',
        'conditions':
            '[{"conditionName":"NoTransientCallbacksCondition"},{"conditionName":"NoPendingFrameCondition"}]',
      });
    });

    test('CombinedCondition serialize - empty condition list', () {
      const CombinedCondition combinedCondition = CombinedCondition(<SerializableWaitCondition>[]);

      expect(combinedCondition.serialize(), <String, String>{
        'conditionName': 'CombinedCondition',
        'conditions': '[]',
      });
    });

    test('CombinedCondition deserialize - empty condition list', () {
      final Map<String, String> jsonMap = <String, String>{
        'conditionName': 'CombinedCondition',
        'conditions': '[]',
      };
      final CombinedCondition condition = CombinedCondition.deserialize(jsonMap);
      expect(condition.conditions, equals(<SerializableWaitCondition>[]));
      expect(condition.serialize(), equals(jsonMap));
    });

    test('CombinedCondition deserialize', () {
      final Map<String, String> jsonMap = <String, String>{
        'conditionName': 'CombinedCondition',
        'conditions':
            '[{"conditionName":"NoPendingFrameCondition"},{"conditionName":"NoTransientCallbacksCondition"}]',
      };
      final CombinedCondition condition = CombinedCondition.deserialize(jsonMap);
      expect(
        condition.conditions,
        equals(<SerializableWaitCondition>[const NoPendingFrame(), const NoTransientCallbacks()]),
      );
      expect(condition.serialize(), jsonMap);
    });

    test('CombinedCondition deserialize - no condition list', () {
      final CombinedCondition condition = CombinedCondition.deserialize(<String, String>{
        'conditionName': 'CombinedCondition',
      });
      expect(condition.conditions, equals(<SerializableWaitCondition>[]));
      expect(condition.serialize(), <String, String>{
        'conditionName': 'CombinedCondition',
        'conditions': '[]',
      });
    });

    test('CombinedCondition deserialize error', () {
      expect(
        () => CombinedCondition.deserialize(<String, String>{'conditionName': 'Unknown'}),
        throwsA(
          predicate<SerializationException>(
            (SerializationException e) =>
                e.message ==
                'Error occurred during deserializing the CombinedCondition JSON string: {conditionName: Unknown}',
          ),
        ),
      );
    });

    test('CombinedCondition deserialize error - Unknown condition type', () {
      expect(
        () {
          return CombinedCondition.deserialize(<String, String>{
            'conditionName': 'CombinedCondition',
            'conditions':
                '[{"conditionName":"UnknownCondition"},{"conditionName":"NoTransientCallbacksCondition"}]',
          });
        },
        throwsA(
          predicate<SerializationException>(
            (SerializationException e) =>
                e.message ==
                'Unsupported wait condition UnknownCondition in the JSON string {conditionName: UnknownCondition}',
          ),
        ),
      );
    });
  });
}
