// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/deferred_component.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/flutter_manifest.dart';

import '../../src/common.dart';

void main() {
  group('DeferredComponent basics', () {
    testWithoutContext('constructor sets values', () {
      final DeferredComponent component = DeferredComponent(
        name: 'bestcomponent',
        libraries: <String>['lib1', 'lib2'],
        assets: <AssetsEntry>[
          AssetsEntry(uri: Uri.file('asset1')),
          AssetsEntry(uri: Uri.file('asset2')),
        ],
      );
      expect(component.name, 'bestcomponent');
      expect(component.libraries, <String>['lib1', 'lib2']);
      expect(component.assets, <AssetsEntry>[
        AssetsEntry(uri: Uri.file('asset1')),
        AssetsEntry(uri: Uri.file('asset2')),
      ]);
    });

    testWithoutContext('assignLoadingUnits selects the needed loading units and sets assigned', () {
      final DeferredComponent component = DeferredComponent(
        name: 'bestcomponent',
        libraries: <String>['lib1', 'lib2'],
        assets: <AssetsEntry>[
          AssetsEntry(uri: Uri.file('asset1')),
          AssetsEntry(uri: Uri.file('asset2')),
        ],
      );
      expect(component.libraries, <String>['lib1', 'lib2']);
      expect(component.assigned, false);
      expect(component.loadingUnits, null);

      final List<LoadingUnit> loadingUnits1 = <LoadingUnit>[
        LoadingUnit(id: 2, path: 'path/to/so.so', libraries: <String>['lib1', 'lib4']),
        LoadingUnit(id: 3, path: 'path/to/so.so', libraries: <String>['lib2', 'lib5']),
        LoadingUnit(id: 4, path: 'path/to/so.so', libraries: <String>['lib6', 'lib7']),
      ];

      component.assignLoadingUnits(loadingUnits1);

      expect(component.assigned, true);
      expect(component.loadingUnits, hasLength(2));
      expect(component.loadingUnits, contains(loadingUnits1[0]));
      expect(component.loadingUnits, contains(loadingUnits1[1]));
      expect(component.loadingUnits, isNot(contains(loadingUnits1[2])));

      final List<LoadingUnit> loadingUnits2 = <LoadingUnit>[
        LoadingUnit(id: 2, path: 'path/to/so.so', libraries: <String>['lib1', 'lib2']),
        LoadingUnit(id: 3, path: 'path/to/so.so', libraries: <String>['lib5', 'lib6']),
        LoadingUnit(id: 4, path: 'path/to/so.so', libraries: <String>['lib7', 'lib8']),
      ];
      // Can reassign loading units.
      component.assignLoadingUnits(loadingUnits2);

      expect(component.assigned, true);
      expect(component.loadingUnits, hasLength(1));
      expect(component.loadingUnits, contains(loadingUnits2[0]));
      expect(component.loadingUnits, isNot(contains(loadingUnits2[1])));
      expect(component.loadingUnits, isNot(contains(loadingUnits2[2])));

      component.assignLoadingUnits(<LoadingUnit>[]);

      expect(component.assigned, true);
      expect(component.loadingUnits, hasLength(0));
    });

    testWithoutContext('toString produces correct string for unassigned component', () {
      final DeferredComponent component = DeferredComponent(
        name: 'bestcomponent',
        libraries: <String>['lib1', 'lib2'],
        assets: <AssetsEntry>[
          AssetsEntry(uri: Uri.file('asset1')),
          AssetsEntry(uri: Uri.file('asset2')),
        ],
      );
      expect(
        component.toString(),
        '\nDeferredComponent: bestcomponent\n  Libraries:\n    - lib1\n    - lib2\n  Assets:\n    - asset1\n    - asset2',
      );
    });

    testWithoutContext('toString produces correct string for assigned component', () {
      final DeferredComponent component = DeferredComponent(
        name: 'bestcomponent',
        libraries: <String>['lib1', 'lib2'],
        assets: <AssetsEntry>[
          AssetsEntry(uri: Uri.file('asset1')),
          AssetsEntry(uri: Uri.file('asset2')),
        ],
      );
      component.assignLoadingUnits(<LoadingUnit>[
        LoadingUnit(id: 2, libraries: <String>['lib1']),
      ]);
      expect(
        component.toString(),
        '\nDeferredComponent: bestcomponent\n  Libraries:\n    - lib1\n    - lib2\n  LoadingUnits:\n    - 2\n  Assets:\n    - asset1\n    - asset2',
      );
    });
  });

  group('LoadingUnit basics', () {
    testWithoutContext('constructor sets values', () {
      final LoadingUnit unit = LoadingUnit(
        id: 2,
        path: 'path/to/so.so',
        libraries: <String>['lib1', 'lib4'],
      );
      expect(unit.id, 2);
      expect(unit.path, 'path/to/so.so');
      expect(unit.libraries, <String>['lib1', 'lib4']);
    });

    testWithoutContext('toString produces correct string', () {
      final LoadingUnit unit = LoadingUnit(
        id: 2,
        path: 'path/to/so.so',
        libraries: <String>['lib1', 'lib4'],
      );
      expect(unit.toString(), '\nLoadingUnit 2\n  Libraries:\n  - lib1\n  - lib4');
    });

    testWithoutContext('equalsIgnoringPath works for various input', () {
      final LoadingUnit unit1 = LoadingUnit(
        id: 2,
        path: 'path/to/so.so',
        libraries: <String>['lib1', 'lib4'],
      );
      final LoadingUnit unit2 = LoadingUnit(
        id: 2,
        path: 'path/to/other/so.so',
        libraries: <String>['lib1', 'lib4'],
      );
      final LoadingUnit unit3 = LoadingUnit(
        id: 1,
        path: 'path/to/other/so.so',
        libraries: <String>['lib1', 'lib4'],
      );
      final LoadingUnit unit4 = LoadingUnit(
        id: 1,
        path: 'path/to/other/so.so',
        libraries: <String>['lib1'],
      );
      final LoadingUnit unit5 = LoadingUnit(
        id: 1,
        path: 'path/to/other/so.so',
        libraries: <String>['lib2'],
      );
      final LoadingUnit unit6 = LoadingUnit(
        id: 1,
        path: 'path/to/other/so.so',
        libraries: <String>['lib1', 'lib5'],
      );
      expect(unit1.equalsIgnoringPath(unit2), true);
      expect(unit2.equalsIgnoringPath(unit3), false);
      expect(unit3.equalsIgnoringPath(unit4), false);
      expect(unit4.equalsIgnoringPath(unit5), false);
      expect(unit5.equalsIgnoringPath(unit6), false);
    });

    testWithoutContext('parseLoadingUnitManifest parses single manifest file', () {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final File manifest = fileSystem.file('/manifest.json');
      manifest.createSync(recursive: true);
      manifest.writeAsStringSync(r'''
{ "loadingUnits": [
{ "id": 1, "path": "\/arm64-v8a\/app.so", "libraries": [
"dart:core"]}
,
{ "id": 2, "path": "\/arm64-v8a\/app.so-2.part.so", "libraries": [
"lib2"]}
,
{ "id": 3, "path": "\/arm64-v8a\/app.so-3.part.so", "libraries": [
"lib3", "lib4"]}
] }

''', flush: true);
      final List<LoadingUnit> loadingUnits = LoadingUnit.parseLoadingUnitManifest(
        manifest,
        BufferLogger.test(),
      );
      expect(loadingUnits.length, 2); // base module (id 1) is not parsed.
      expect(loadingUnits[0].id, 2);
      expect(loadingUnits[0].path, '/arm64-v8a/app.so-2.part.so');
      expect(loadingUnits[0].libraries.length, 1);
      expect(loadingUnits[0].libraries[0], 'lib2');
      expect(loadingUnits[1].id, 3);
      expect(loadingUnits[1].path, '/arm64-v8a/app.so-3.part.so');
      expect(loadingUnits[1].libraries.length, 2);
      expect(loadingUnits[1].libraries[0], 'lib3');
      expect(loadingUnits[1].libraries[1], 'lib4');
    });

    testWithoutContext('parseLoadingUnitManifest returns empty when manifest is invalid JSON', () {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final File manifest = fileSystem.file('/manifest.json');
      manifest.createSync(recursive: true);
      // invalid due to missing closing brace `}`
      manifest.writeAsStringSync(r'''
{ "loadingUnits": [
{ "id": 1, "path": "\/arm64-v8a\/app.so", "libraries": [
"dart:core"]}
,
{ "id": 2, "path": "\/arm64-v8a\/app.so-2.part.so", "libraries": [
"lib2"]}
,
{ "id": 3, "path": "\/arm64-v8a\/app.so-3.part.so", "libraries": [
"lib3", "lib4"]}
]

''', flush: true);
      final List<LoadingUnit> loadingUnits = LoadingUnit.parseLoadingUnitManifest(
        manifest,
        BufferLogger.test(),
      );
      expect(loadingUnits.length, 0);
      expect(loadingUnits.isEmpty, true);
    });

    testWithoutContext('parseLoadingUnitManifest does not exist', () {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final File manifest = fileSystem.file('/manifest.json');
      if (manifest.existsSync()) {
        manifest.deleteSync(recursive: true);
      }
      final List<LoadingUnit> loadingUnits = LoadingUnit.parseLoadingUnitManifest(
        manifest,
        BufferLogger.test(),
      );
      expect(loadingUnits.length, 0);
      expect(loadingUnits.isEmpty, true);
    });

    testWithoutContext('parseGeneratedLoadingUnits parses all abis if no abis provided', () {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final File manifest1 = fileSystem.file('/test-abi1/manifest.json');
      manifest1.createSync(recursive: true);
      manifest1.writeAsStringSync(r'''
{ "loadingUnits": [
{ "id": 1, "path": "\/test-abi1\/app.so", "libraries": [
"dart:core"]}
,
{ "id": 2, "path": "\/test-abi1\/app.so-2.part.so", "libraries": [
"lib2"]}
,
{ "id": 3, "path": "\/test-abi1\/app.so-3.part.so", "libraries": [
"lib3", "lib4"]}
] }

''', flush: true);
      final File manifest2 = fileSystem.file('/test-abi2/manifest.json');
      manifest2.createSync(recursive: true);
      manifest2.writeAsStringSync(r'''
{ "loadingUnits": [
{ "id": 1, "path": "\/test-abi2\/app.so", "libraries": [
"dart:core"]}
,
{ "id": 2, "path": "\/test-abi2\/app.so-2.part.so", "libraries": [
"lib2"]}
,
{ "id": 3, "path": "\/test-abi2\/app.so-3.part.so", "libraries": [
"lib3", "lib4"]}
] }

''', flush: true);
      final List<LoadingUnit> loadingUnits = LoadingUnit.parseGeneratedLoadingUnits(
        fileSystem.directory('/'),
        BufferLogger.test(),
      );
      expect(loadingUnits.length, 4); // base module (id 1) is not parsed.

      expect(loadingUnits[0].id, 2);
      expect(loadingUnits[0].path, '/test-abi2/app.so-2.part.so');
      expect(loadingUnits[0].libraries.length, 1);
      expect(loadingUnits[0].libraries[0], 'lib2');
      expect(loadingUnits[1].id, 3);
      expect(loadingUnits[1].path, '/test-abi2/app.so-3.part.so');
      expect(loadingUnits[1].libraries.length, 2);
      expect(loadingUnits[1].libraries[0], 'lib3');
      expect(loadingUnits[1].libraries[1], 'lib4');

      expect(loadingUnits[2].id, 2);
      expect(loadingUnits[2].path, '/test-abi1/app.so-2.part.so');
      expect(loadingUnits[2].libraries.length, 1);
      expect(loadingUnits[2].libraries[0], 'lib2');
      expect(loadingUnits[3].id, 3);
      expect(loadingUnits[3].path, '/test-abi1/app.so-3.part.so');
      expect(loadingUnits[3].libraries.length, 2);
      expect(loadingUnits[3].libraries[0], 'lib3');
      expect(loadingUnits[3].libraries[1], 'lib4');
    });

    testWithoutContext('parseGeneratedLoadingUnits only parses provided abis', () {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final File manifest1 = fileSystem.file('/test-abi1/manifest.json');
      manifest1.createSync(recursive: true);
      manifest1.writeAsStringSync(r'''
{ "loadingUnits": [
{ "id": 1, "path": "\/test-abi1\/app.so", "libraries": [
"dart:core"]}
,
{ "id": 2, "path": "\/test-abi1\/app.so-2.part.so", "libraries": [
"lib2"]}
,
{ "id": 3, "path": "\/test-abi1\/app.so-3.part.so", "libraries": [
"lib3", "lib4"]}
] }

''', flush: true);
      final File manifest2 = fileSystem.file('/test-abi2/manifest.json');
      manifest2.createSync(recursive: true);
      manifest2.writeAsStringSync(r'''
{ "loadingUnits": [
{ "id": 1, "path": "\/test-abi2\/app.so", "libraries": [
"dart:core"]}
,
{ "id": 2, "path": "\/test-abi2\/app.so-2.part.so", "libraries": [
"lib2"]}
,
{ "id": 3, "path": "\/test-abi2\/app.so-3.part.so", "libraries": [
"lib3", "lib4"]}
] }

''', flush: true);
      final List<LoadingUnit> loadingUnits = LoadingUnit.parseGeneratedLoadingUnits(
        fileSystem.directory('/'),
        BufferLogger.test(),
        abis: <String>['test-abi2'],
      );
      expect(loadingUnits.length, 2); // base module (id 1) is not parsed.

      expect(loadingUnits[0].id, 2);
      expect(loadingUnits[0].path, '/test-abi2/app.so-2.part.so');
      expect(loadingUnits[0].libraries.length, 1);
      expect(loadingUnits[0].libraries[0], 'lib2');
      expect(loadingUnits[1].id, 3);
      expect(loadingUnits[1].path, '/test-abi2/app.so-3.part.so');
      expect(loadingUnits[1].libraries.length, 2);
      expect(loadingUnits[1].libraries[0], 'lib3');
      expect(loadingUnits[1].libraries[1], 'lib4');
    });

    testWithoutContext('parseGeneratedLoadingUnits returns empty when no manifest files exist', () {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final List<LoadingUnit> loadingUnits = LoadingUnit.parseGeneratedLoadingUnits(
        fileSystem.directory('/'),
        BufferLogger.test(),
      );
      expect(loadingUnits.isEmpty, true);
      expect(loadingUnits.length, 0);
    });
  });
}
