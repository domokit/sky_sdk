// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:engine_tool/src/typed_json.dart';
import 'package:test/test.dart';

void main() {
  group('JsonObject.string', () {
    test('returns string value', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{'key': 'value'});
      expect(jsonObject.string('key'), 'value');
    });

    test('throws due to missing key', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{'key': 'value'});
      expect(
        () => jsonObject.string('missing'),
        throwsA(const isInstanceOf<MissingKeyJsonReadException>()),
      );
    });

    test('throws due to wrong type', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{'key': 42});
      expect(
        () => jsonObject.string('key'),
        throwsA(const isInstanceOf<InvalidTypeJsonReadException>()),
      );
    });
  });

  group('JsonObject.integer', () {
    test('returns integer value', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{'key': 42});
      expect(jsonObject.integer('key'), 42);
    });

    test('throws due to missing key', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{'key': 42});
      expect(
        () => jsonObject.integer('missing'),
        throwsA(const isInstanceOf<MissingKeyJsonReadException>()),
      );
    });

    test('throws due to wrong type', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{'key': 'value'});
      expect(
        () => jsonObject.integer('key'),
        throwsA(const isInstanceOf<InvalidTypeJsonReadException>()),
      );
    });
  });

  group('JsonObject.boolean', () {
    test('returns boolean value', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{'key': true});
      expect(jsonObject.boolean('key'), true);
    });

    test('throws due to missing key', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{'key': true});
      expect(
        () => jsonObject.boolean('missing'),
        throwsA(const isInstanceOf<MissingKeyJsonReadException>()),
      );
    });

    test('throws due to wrong type', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{'key': 'value'});
      expect(
        () => jsonObject.boolean('key'),
        throwsA(const isInstanceOf<InvalidTypeJsonReadException>()),
      );
    });
  });

  group('JsonObject.stringList', () {
    test('returns string list value', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{
        'key': <Object?>['value1', 'value2'],
      });
      expect(jsonObject.stringList('key'), <String>['value1', 'value2']);
    });

    test('throws due to missing key', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{
        'key': <Object?>['value1', 'value2'],
      });
      expect(
        () => jsonObject.stringList('missing'),
        throwsA(const isInstanceOf<MissingKeyJsonReadException>()),
      );
    });

    test('throws due to wrong type', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{'key': 'value'});
      expect(
        () => jsonObject.stringList('key'),
        throwsA(const isInstanceOf<InvalidTypeJsonReadException>()),
      );
    });

    test('throws due to wrong element type', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{
        'key': <Object?>['value1', 42],
      });
      expect(
        () => jsonObject.stringList('key'),
        throwsA(const isInstanceOf<InvalidTypeJsonReadException>()),
      );
    });
  });

  group('JsonObject.stringOrNull', () {
    test('returns string value', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{'key': 'value'});
      expect(jsonObject.stringOrNull('key'), 'value');
    });

    test('returns null due to missing key', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{'key': 'value'});
      expect(jsonObject.stringOrNull('missing'), isNull);
    });

    test('throws due to wrong type', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{'key': 42});
      expect(
        () => jsonObject.stringOrNull('key'),
        throwsA(const isInstanceOf<InvalidTypeJsonReadException>()),
      );
    });
  });

  group('JsonObject.integerOrNull', () {
    test('returns integer value', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{'key': 42});
      expect(jsonObject.integerOrNull('key'), 42);
    });

    test('returns null due to missing key', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{'key': 42});
      expect(jsonObject.integerOrNull('missing'), isNull);
    });

    test('throws due to wrong type', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{'key': 'value'});
      expect(
        () => jsonObject.integerOrNull('key'),
        throwsA(const isInstanceOf<InvalidTypeJsonReadException>()),
      );
    });
  });

  group('JsonObject.booleanOrNull', () {
    test('returns boolean value', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{'key': true});
      expect(jsonObject.booleanOrNull('key'), true);
    });

    test('returns null due to missing key', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{'key': true});
      expect(jsonObject.booleanOrNull('missing'), isNull);
    });

    test('throws due to wrong type', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{'key': 'value'});
      expect(
        () => jsonObject.booleanOrNull('key'),
        throwsA(const isInstanceOf<InvalidTypeJsonReadException>()),
      );
    });
  });

  group('JsonObject.stringListOrNull', () {
    test('returns string list value', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{
        'key': <Object?>['value1', 'value2'],
      });
      expect(jsonObject.stringListOrNull('key'), <String>['value1', 'value2']);
    });

    test('returns null due to missing key', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{
        'key': <Object?>['value1', 'value2'],
      });
      expect(jsonObject.stringListOrNull('missing'), isNull);
    });

    test('throws due to wrong type', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{'key': 'value'});
      expect(
        () => jsonObject.stringListOrNull('key'),
        throwsA(const isInstanceOf<InvalidTypeJsonReadException>()),
      );
    });

    test('throws due to wrong element type', () {
      const JsonObject jsonObject = JsonObject(<String, Object?>{
        'key': <Object?>['value1', 42],
      });
      expect(
        () => jsonObject.stringListOrNull('key'),
        throwsA(const isInstanceOf<InvalidTypeJsonReadException>()),
      );
    });
  });

  group('JsonObject.objectOrNull', () {
    test('returns object value', () {
      const jsonObject = JsonObject({
        'key': {'nestedKey': 'nestedValue'},
      });
      expect(jsonObject.objectOrNull('key'), const JsonObject({'nestedKey': 'nestedValue'}));
    });

    test('returns null due to missing key', () {
      const jsonObject = JsonObject({
        'key': {'nestedKey': 'nestedValue'},
      });
      expect(jsonObject.objectOrNull('missing'), isNull);
    });

    test('throws due to wrong type', () {
      const jsonObject = JsonObject({'key': 'value'});
      expect(
        () => jsonObject.objectOrNull('key'),
        throwsA(const isInstanceOf<InvalidTypeJsonReadException>()),
      );
    });
  });

  group('JsonObject.map', () {
    test('returns multiple fields', () {
      final (String name, int age, bool isStudent) = const JsonObject(<String, Object?>{
        'name': 'Alice',
        'age': 42,
        'isStudent': true,
      }).map((JsonObject json) {
        return (json.string('name'), json.integer('age'), json.boolean('isStudent'));
      });

      expect(name, 'Alice');
      expect(age, 42);
      expect(isStudent, true);
    });

    test('throws due to missing keys', () {
      try {
        const JsonObject(<String, Object?>{'age': 42}).map((JsonObject json) {
          return (json.string('name'), json.integer('age'), json.boolean('isStudent'));
        });
        fail('Expected JsonMapException');
      } on JsonMapException catch (e) {
        expect(
          e.exceptions
              .map((JsonReadException e) => (e as MissingKeyJsonReadException).key)
              .toList(),
          containsAllInOrder(<String>['name', 'isStudent']),
        );
      }
    });

    test('throws due to wrong types', () {
      try {
        const JsonObject(<String, Object?>{'name': 42, 'age': '42', 'isStudent': false}).map((
          JsonObject json,
        ) {
          return (json.string('name'), json.integer('age'), json.boolean('isStudent'));
        });
        fail('Expected JsonMapException');
      } on JsonMapException catch (e) {
        expect(
          e.exceptions
              .map(
                (JsonReadException e) => switch (e) {
                  final InvalidTypeJsonReadException e => e.key,
                  final MissingKeyJsonReadException e => e.key,
                  _ => throw StateError('Unexpected exception type: $e'),
                },
              )
              .toList(),
          containsAllInOrder(<String>['name', 'age']),
        );
      }
    });

    test('allows a default with onError', () {
      final (String name, int age, bool isStudent) = const JsonObject(<String, Object?>{
        'name': 'Alice',
        'age': 42,
        'isStudent': 'true',
      }).map(
        (JsonObject json) {
          return (json.string('name'), json.integer('age'), json.boolean('isStudent'));
        },
        onError: expectAsync2((_, JsonMapException e) {
          expect(
            e.exceptions
                .map(
                  (JsonReadException e) => switch (e) {
                    final InvalidTypeJsonReadException e => e.key,
                    final MissingKeyJsonReadException e => e.key,
                    _ => throw StateError('Unexpected exception type: $e'),
                  },
                )
                .toList(),
            <String>['isStudent'],
          );
          return ('Bob', 0, false);
        }),
      );

      expect(name, 'Bob');
      expect(age, 0);
      expect(isStudent, false);
    });

    test('disallows a return type of Future<*>', () {
      expect(() {
        const JsonObject(<String, Object?>{'name': 'Alice', 'age': 42, 'isStudent': true}).map((
          JsonObject json,
        ) async {
          return (json.string('name'), json.integer('age'), json.boolean('isStudent'));
        });
      }, throwsA(const isInstanceOf<ArgumentError>()));
    });
  });
}
